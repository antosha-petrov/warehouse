using System.Collections.ObjectModel;
using System.Net.Http.Json;
using System.Windows.Input;
using Models;

namespace WarehouseApp.ViewModel
{
    public class MainPageViewModel
    {
        public ObservableCollection<CardViewModel> Items { get; }

        private readonly HttpClient _httpClient;

#pragma warning disable IDE0052
        private readonly Timer _timer;
#pragma warning restore IDE0052

        private HttpBackgroundService backgroundService { get; set; }

        public ICommand NavigateToOrder { get; }

        public MainPageViewModel()
        {
            _httpClient = new HttpClient { BaseAddress = new Uri("http://Honor:5298/") };
            Items = new ObservableCollection<CardViewModel>();
            NavigateToOrder = new Command(() => Shell.Current.GoToAsync("//Order"));
            backgroundService = new HttpBackgroundService();
            _timer = new Timer(async _ => await SendHttpRequest(), null, TimeSpan.Zero, TimeSpan.FromSeconds(2));
        }

        // Асинхронный метод для загрузки элементов
        public async Task LoadItemsAsync()
        {
            var appState = await AppState.GetInstanceAsync(); // Получаем инстанс AppState асинхронно

            var updatedItems = appState.Goods?.Select(goods => new CardViewModel(goods)) ?? Enumerable.Empty<CardViewModel>();

            Items.Clear();
            foreach (var item in updatedItems)
            {
                Items.Add(item);
            }
        }

        // Асинхронный метод для обновления элементов
        public async Task RefreshItemsAsync()
        {
            var appState = await AppState.GetInstanceAsync(); // Получаем инстанс AppState асинхронно

            var updatedItems = appState.Goods?.Select(goods => new CardViewModel(goods)) ?? Enumerable.Empty<CardViewModel>();

            Items.Clear();
            foreach (var item in updatedItems)
            {
                Items.Add(item);
            }
        }

        private async Task SendHttpRequest()
        {
            HttpResponseMessage response = await _httpClient.GetAsync("orders/get/app");
            var _lastOrder = await response.Content.ReadFromJsonAsync<Order>();
            if (_lastOrder == null || _lastOrder.Items == null) return;

            var appState = await AppState.GetInstanceAsync();

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
}
