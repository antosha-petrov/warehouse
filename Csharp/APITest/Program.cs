using System.Text;
using Newtonsoft.Json;

class Program
{
    private static readonly HttpClient _httpClient = new() { BaseAddress = new Uri("http://192.168.1.56:5298/") };

    static async Task Main()
    {
        Console.WriteLine("Тест API запущен...");

        await TestCreateOrder();
        await TestUpdateOrder();

        Console.WriteLine("Тестирование завершено.");
        Console.ReadLine();
    }

    private static async Task TestCreateOrder()
    {
        Console.WriteLine("\n[POST] Создание нового заказа...");
        var response = await _httpClient.PostAsync("orders/create", null);
        var content = await response.Content.ReadAsStringAsync();
        Console.WriteLine($"Статус: {response.StatusCode}\nОтвет: {content}");
    }

    private static async Task TestUpdateOrder()
    {
        Console.WriteLine("\n[PUT] Обновление количества товаров...");
        var data = new int[] { 1, 2, 3, 4, 5 };
        var json = JsonConvert.SerializeObject(data);
        var content = new StringContent(json, Encoding.UTF8, "application/json");

        var response = await _httpClient.PutAsync("orders/update/count", content);
        var responseContent = await response.Content.ReadAsStringAsync();
        Console.WriteLine($"Статус: {response.StatusCode}\nОтвет: {responseContent}");
    }
}
