using System.Net.Http.Json;
using System.Text;
using Models;
using Newtonsoft.Json;

public class AppState
{
    private static AppState? _instance;
    private static readonly object _lock = new();
    private readonly HttpClient _httpClient;

#pragma warning disable IDE0052
    private readonly Timer _timer;
#pragma warning restore IDE0052

    private AppState()
    {
        _httpClient = new HttpClient { BaseAddress = new Uri("http://192.168.1.57:5298/") };
        _timer = new Timer(async _ => await RefreshStatus(), null, TimeSpan.Zero, TimeSpan.FromSeconds(2));
    }

    public static async Task<AppState> GetInstanceAsync()
    {
        // Используем блокировку для безопасной инициализации в многопоточном приложении
        if (_instance == null)
        {
            lock (_lock)
            {
                if (_instance == null)
                {
                    _instance = new AppState();
                }
            }
            // Асинхронная инициализация состояния
            await _instance.InitializeStateAsync();
        }

        return _instance;
    }

    public List<OrderItem>? Goods { get; private set; }

    private async Task InitializeStateAsync()
    {
        try
        {
            var createOrderResponse = await CreateOrderAsync();
            if (createOrderResponse.IsSuccessStatusCode)
            {
                var newOrder = JsonConvert.DeserializeObject<Order>(await createOrderResponse.Content.ReadAsStringAsync());
                Goods = newOrder?.Items ?? new List<OrderItem>();
            }
        }
        catch (Exception ex)
        {
            Console.WriteLine($"Ошибка при инициализации состояния: {ex.Message}");
        }
    }

    private async Task<HttpResponseMessage> CreateOrderAsync()
    {
        try
        {
            return await _httpClient.PostAsync("orders/create", null);
        }
        catch (Exception ex)
        {
            Console.WriteLine($"Ошибка при создании заказа: {ex.Message}");
            throw;
        }
    }

    public async Task<HttpResponseMessage> EditOrderAsync(int[] nums)
    {
        var content = new StringContent(JsonConvert.SerializeObject(nums), Encoding.UTF8, "application/json");
        return await _httpClient.PutAsync("orders/update/count", content);
    }

    private async Task RefreshStatus()
    {
        try
        {
            var appState = await AppState.GetInstanceAsync();

            if (appState.Goods != null)
            {
                HttpResponseMessage response = await _httpClient.GetAsync("orders/get/app");
                response.EnsureSuccessStatusCode();
                var _lastOrder = await response.Content.ReadFromJsonAsync<Order>();
                if (_lastOrder == null || _lastOrder.Items == null) return;

                foreach (var item in appState.Goods!)
                {
                    var matchingOrderItem = _lastOrder.Items.FirstOrDefault(o => o.Goods.Name == item.Goods.Name);
                    if (matchingOrderItem != null)
                    {
                        item.Shelf = matchingOrderItem.Shelf;
                        item.Cell = matchingOrderItem.Cell;
                        item.Rack = matchingOrderItem.Rack;
                    }
                }
            }
        }
        catch (Exception ex)
        {
            Console.WriteLine(ex.Message);
        }
    }
}
